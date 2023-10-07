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

network DADCA_ACK_n50_u16_a8000m2_5
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
        @display("i=misc/sensor;p=2181.08,7925.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2609.95,7961.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3401.46,7764.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4873.22,7809.20");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5792.22,7804.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=427.47,7524.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=904.11,7516.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2981.78,7738.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4620.03,7523.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5219.89,7526.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1974.66,7370.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2678.33,7452.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3237.08,7380.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3952.38,7299.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5973.30,7484.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=166.70,7187.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=649.54,7100.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1440.33,7177.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2386.45,7038.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2839.16,7122.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4310.64,7218.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5393.05,7204.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1907.42,6967.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3130.84,6869.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3687.89,6965.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4116.17,6819.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4894.98,7000.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5623.64,6880.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5991.84,6823.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=758.51,6519.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1017.43,6759.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1774.07,6545.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2621.08,6543.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4656.49,6679.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5232.83,6579.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=261.23,6386.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1304.41,6475.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3488.87,6428.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4001.35,6455.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5926.89,6477.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1072.10,6120.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2306.08,6230.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4542.85,6149.50");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5416.38,6033.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5796.12,6104.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=362.62,6010.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=740.27,5942.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1545.72,5961.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2045.85,5865.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2788.50,5926.70");
    }

}

