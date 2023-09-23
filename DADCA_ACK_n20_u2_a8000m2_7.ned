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

network DADCA_ACK_n20_u2_a8000m2_7
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
        @display("i=misc/sensor;p=608.33,7999.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1300.93,7926.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1835.08,7883.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2758.97,7830.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1065.32,7535.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2195.23,7639.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3302.70,7517.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3688.00,7526.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4252.47,7544.27");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4884.47,7714.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5356.54,7661.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5739.15,7719.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1578.62,7450.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2741.80,7394.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5852.93,7334.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=76.16,7110.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=636.31,7151.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1273.60,7037.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1936.72,7217.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3737.46,7162.32");
    }

}

