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

network DADCA_ACK_n100_a8000m2_4
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
        @display("i=misc/sensor;p=179.35,7818.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=874.77,7911.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2516.04,7885.05");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2987.29,7892.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3642.82,7912.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3997.03,7987.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1156.41,7542.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1707.78,7507.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2238.87,7570.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3501.30,7569.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4509.86,7701.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5062.30,7642.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5592.13,7618.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=592.49,7310.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2734.93,7401.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3092.02,7481.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4075.42,7422.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4890.41,7288.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=283.00,7138.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1074.80,7160.96");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1739.65,7150.43");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3575.11,7100.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5421.22,7235.61");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5839.58,7196.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=770.73,6808.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2348.90,6920.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3024.07,6903.97");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4106.53,6979.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4527.50,6894.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5120.15,6950.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5966.67,6862.13");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=422.45,6516.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1183.27,6753.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2148.24,6589.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2720.22,6667.82");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3603.03,6736.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4075.58,6619.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5629.05,6752.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=908.26,6470.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1388.02,6344.95");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1743.78,6401.35");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3154.26,6467.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5010.35,6344.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=552.70,6127.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2055.38,6214.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2766.10,6123.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3952.46,6258.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4489.87,6060.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5328.30,6130.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5754.58,6233.65");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1164.77,5797.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1841.07,5830.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2343.38,5942.96");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3569.43,5911.31");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=286.00,5599.44");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1446.99,5586.08");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2111.66,5542.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4349.45,5662.88");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4764.37,5654.27");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5139.29,5728.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5529.39,5640.97");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1032.44,5467.90");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3269.47,5397.63");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3960.58,5452.81");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1620.38,5032.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2043.28,5173.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2654.81,5199.54");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4455.37,5169.36");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4834.92,5177.99");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5337.71,5181.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5695.37,5107.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=560.00,4972.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2664.14,4838.15");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3837.06,5009.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=947.68,4729.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1761.68,4582.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2133.24,4748.86");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3344.03,4719.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3946.40,4620.44");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4485.08,4550.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5439.05,4728.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=227.30,4394.50");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=878.06,4342.81");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1288.09,4457.22");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2307.33,4385.29");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2878.00,4438.67");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3567.94,4413.86");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4846.94,4429.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5736.85,4451.42");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1957.67,4223.01");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4119.04,4137.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4642.97,4081.58");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5223.52,4114.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=519.58,3863.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=986.21,3954.61");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1530.55,3998.66");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2036.17,3851.09");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2639.45,3908.26");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3237.28,3848.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3595.50,3893.35");
    }

}

