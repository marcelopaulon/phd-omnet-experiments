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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=394.55,7800.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=845.87,7888.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1185.74,7791.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1472.88,7998.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2222.01,7809.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3618.78,7953.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4742.43,7842.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5370.08,7754.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=138.24,7516.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=683.02,7567.10");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2538.38,7560.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2879.30,7736.72");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3557.17,7577.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3902.61,7704.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4255.98,7676.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5741.13,7510.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1330.12,7386.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1907.21,7502.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2256.66,7300.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2899.16,7354.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4455.11,7307.65");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4799.27,7444.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1000.84,7173.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3844.31,7194.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5075.90,7187.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5492.18,7155.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=291.54,6997.47");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2417.30,6936.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2928.12,6952.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3401.64,6922.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4338.37,6916.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=801.34,6668.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1460.92,6665.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1985.38,6653.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2615.42,6574.87");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3258.41,6552.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3870.78,6615.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4883.68,6680.45");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5278.88,6533.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5899.70,6727.78");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=472.95,6413.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1399.87,6304.96");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2179.69,6344.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3608.28,6306.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=88.59,6100.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=789.61,6185.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1834.47,6253.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2997.64,6105.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4290.87,6115.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4643.15,6130.62");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5585.62,6230.25");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1110.12,5936.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3931.52,5977.04");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5107.31,5955.32");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=6.29,5586.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=703.23,5611.60");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1632.94,5751.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2244.37,5764.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2699.07,5660.15");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3158.86,5626.50");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3717.07,5618.32");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4107.86,5670.95");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4825.33,5627.12");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5778.50,5698.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=350.02,5511.92");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1098.33,5343.70");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1799.92,5367.32");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2189.91,5331.66");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3505.26,5293.02");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4470.90,5397.24");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=526.89,5031.16");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1378.79,5030.42");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2800.22,5155.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3884.32,5074.49");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4864.85,5105.03");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5259.51,5213.99");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5671.81,5164.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=58.03,4956.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2177.53,4809.76");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2615.33,4830.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3277.28,4986.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4298.31,4918.14");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=192.46,4595.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1094.75,4636.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1543.86,4661.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3577.76,4612.69");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4057.55,4599.43");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4650.10,4741.53");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5121.69,4677.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5493.27,4762.65");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=576.86,4521.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2394.38,4514.72");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2924.37,4425.55");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5484.16,4344.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5931.40,4435.76");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1200.08,4242.93");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2206.72,4083.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3433.45,4197.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3954.23,4249.10");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4350.96,4148.24");
    }

}

