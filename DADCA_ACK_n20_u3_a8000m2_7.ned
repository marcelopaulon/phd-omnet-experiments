package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_u3_a8000m2_7
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1171.39,7835.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1821.72,7981.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2348.47,7939.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2750.45,7875.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3178.42,7797.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5278.49,7950.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3558.64,7629.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5470.46,7570.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5870.49,7629.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=249.48,7370.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1528.14,7426.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1932.86,7480.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2461.77,7486.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3120.56,7298.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4032.89,7451.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4687.77,7467.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5077.47,7431.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=382.06,7021.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=731.00,7061.07");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1099.59,7032.48");
    }

}

