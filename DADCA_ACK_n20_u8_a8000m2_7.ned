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

network DADCA_ACK_n20_u8_a8000m2_7
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
        @display("i=misc/sensor;p=417.06,7942.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1504.44,7857.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2661.19,7802.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3239.72,7754.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3563.92,7941.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4014.51,7999.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4657.46,7787.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5270.38,7950.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=252.10,7536.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=854.22,7669.52");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1255.05,7506.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3540.62,7548.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4090.14,7588.98");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2117.65,7417.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3009.41,7313.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3838.29,7265.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4274.91,7283.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5127.69,7335.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=647.25,7153.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1654.15,7218.32");
    }

}

