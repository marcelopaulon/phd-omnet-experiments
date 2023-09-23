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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=366.64,7818.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1842.59,7944.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3900.42,7965.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5040.25,7916.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5917.85,7775.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=33.13,7660.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=993.95,7517.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1445.83,7647.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2314.84,7650.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2819.71,7744.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3392.62,7697.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3806.57,7561.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4643.90,7568.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5125.74,7568.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5506.87,7516.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=589.51,7272.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1933.39,7353.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2578.33,7358.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4237.75,7491.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1199.40,7050.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3230.45,7180.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3599.03,7116.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4755.05,7216.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5270.45,7126.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5653.80,7182.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1762.04,6994.61");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2281.60,6865.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2613.69,6996.31");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4058.63,6923.68");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4471.85,6993.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=607.29,6647.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1352.13,6642.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1945.21,6556.19");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3056.97,6658.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3462.58,6558.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4151.16,6580.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5019.45,6762.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5491.03,6557.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5907.25,6687.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=273.56,6443.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=838.80,6383.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2417.96,6450.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4924.70,6363.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1602.36,6146.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3087.47,6259.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3771.99,6176.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4507.34,6022.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5353.84,6188.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5706.18,6210.35");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=457.66,5998.66");
    }

}

