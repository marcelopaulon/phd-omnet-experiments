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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=638.03,7975.96");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1269.08,7959.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3025.47,7759.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3597.27,7917.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4085.29,7838.67");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4745.11,7821.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5780.16,7870.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=743.65,7639.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1172.78,7547.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1629.84,7706.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2727.45,7567.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5273.65,7637.21");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=120.64,7376.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2077.33,7383.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3788.85,7462.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4425.11,7382.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4915.86,7264.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1031.65,7191.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1503.18,7168.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2364.38,7126.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2903.59,7219.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3486.93,7181.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4165.35,7102.04");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5816.71,7231.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=29.17,6779.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=697.41,6978.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1924.53,6957.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2596.63,6853.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3043.26,6824.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4671.23,6793.91");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5117.68,6974.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=377.97,6703.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=920.37,6535.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1373.30,6725.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3633.44,6702.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4195.89,6599.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5504.70,6629.95");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5940.88,6679.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=140.58,6307.39");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2044.15,6452.49");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3103.12,6274.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3525.26,6321.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4842.09,6461.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5181.61,6293.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=509.23,6226.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1105.98,6214.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1685.09,6047.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2444.22,6251.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2760.57,6099.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3944.88,6166.26");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4468.67,6059.36");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5688.70,6064.11");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3097.56,5846.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3693.49,5891.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4982.62,5777.32");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5340.77,5825.64");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=336.77,5596.45");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1133.21,5703.02");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2576.14,5647.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3413.53,5637.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4007.81,5642.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5633.58,5566.62");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=988.33,5328.60");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1667.76,5434.34");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3092.50,5378.78");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3604.59,5306.18");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4530.88,5512.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=156.43,5168.46");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1722.55,5062.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2081.35,5185.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3954.56,5226.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4351.78,5082.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4787.53,5100.74");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5650.66,5035.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5985.54,5201.06");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=686.76,4886.04");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1283.37,5007.13");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2836.30,4963.29");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3417.83,4928.21");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5947.54,4799.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=75.54,4729.83");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2047.25,4744.84");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2370.02,4577.78");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2826.07,4574.73");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3747.50,4746.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4411.05,4642.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5055.10,4683.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5604.70,4617.28");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=475.88,4527.08");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1119.36,4514.85");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1451.50,4362.19");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3323.38,4456.40");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5989.52,4332.16");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=129.66,4124.85");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=787.14,4210.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2093.54,4108.10");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2780.66,4160.54");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3895.13,4194.60");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4426.15,4152.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5149.83,4207.51");
    }

}

