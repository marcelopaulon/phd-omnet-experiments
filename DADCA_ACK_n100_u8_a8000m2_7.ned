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

network DADCA_ACK_n100_u8_a8000m2_7
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
        @display("i=misc/sensor;p=1683.74,7757.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2322.58,7929.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2842.16,7832.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3460.71,7823.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4539.35,7860.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5175.79,7913.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5609.38,7777.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=558.29,7752.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1231.22,7683.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2135.37,7624.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4104.79,7668.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4980.27,7611.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5989.54,7667.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=289.80,7381.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1958.19,7273.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2608.37,7363.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3328.57,7379.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3689.36,7379.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4563.58,7481.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5389.02,7345.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=879.39,7194.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4301.27,7204.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4749.47,7087.03");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5832.99,7047.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=27.45,6940.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1152.26,6960.59");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2535.72,6944.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2895.63,6767.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3199.84,6955.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3566.27,6937.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4085.93,6873.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4477.97,6845.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5276.64,6783.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=758.22,6686.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1152.59,6561.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2117.80,6715.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=330.98,6428.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2489.72,6383.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3789.59,6505.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4832.91,6417.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5849.57,6468.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=614.25,6179.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1391.03,6242.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1913.85,6117.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3162.29,6211.30");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4231.14,6096.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5355.49,6079.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=259.32,5895.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=965.99,6018.44");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1666.84,5815.03");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2160.62,5827.28");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2557.66,6013.38");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3721.38,5956.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4960.95,5984.97");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5912.70,5935.87");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2458.56,5638.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3972.20,5633.64");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4341.76,5605.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5207.35,5600.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=137.73,5334.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=842.95,5488.64");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1259.70,5478.79");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2173.76,5278.97");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2857.98,5366.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3421.12,5345.70");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4828.11,5439.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5618.36,5420.64");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5980.11,5521.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=690.02,5113.52");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1758.17,5098.39");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4071.74,5194.46");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4436.00,5184.84");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=831.19,4791.90");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1263.86,4812.41");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2204.98,4817.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2582.32,4818.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3469.42,4869.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4503.09,4809.37");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5100.56,4998.12");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1672.88,4636.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3079.84,4691.41");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4075.18,4602.70");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4808.83,4621.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5706.62,4778.36");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=201.07,4453.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=650.15,4492.31");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1408.57,4306.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2844.14,4368.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3643.99,4361.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5088.51,4402.64");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5449.45,4531.57");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1032.57,4239.69");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1985.28,4236.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2507.29,4191.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3219.36,4195.77");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4441.46,4204.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4904.31,4101.62");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5852.50,4225.42");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=516.50,3835.36");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=906.76,3823.68");
    }

}

