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

network DADCA_ACK_n20_u16_a8000m2_2
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
        @display("i=misc/sensor;p=502.39,7975.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1110.00,7940.48");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1506.25,7905.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1942.08,7952.63");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2650.68,7856.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3027.50,7759.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3871.21,7964.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4330.79,7658.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4673.77,7557.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5526.57,7519.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1569.87,7481.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2032.97,7396.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2745.88,7453.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3100.28,7335.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3469.29,7498.39");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=315.45,7157.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=866.75,7229.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1525.06,7097.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2414.76,7051.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3887.98,7254.10");
    }

}

