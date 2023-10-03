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

network DADCA_ACK_n100_u16_a8000m2_3
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
        @display("i=misc/sensor;p=1109.35,7985.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2272.50,7891.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2832.07,7762.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3769.55,7901.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4998.65,7978.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=531.60,7616.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1127.13,7627.69");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2296.97,7516.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4129.90,7649.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4505.61,7610.04");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5203.61,7640.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5681.77,7594.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1839.75,7468.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3300.99,7452.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4943.64,7394.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5352.48,7295.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=336.62,7111.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=698.17,7142.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1373.93,7198.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2738.17,7219.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4228.80,7100.38");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5748.39,7091.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=901.90,6809.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1381.85,6834.09");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1942.70,6822.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2503.32,6792.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3080.92,6883.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3737.32,6974.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4576.75,6905.01");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5006.08,6844.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4316.99,6598.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4754.28,6523.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5595.81,6748.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=917.45,6453.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1417.74,6326.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2991.97,6507.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3578.52,6472.35");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5906.38,6330.99");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=836.10,6037.76");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1997.04,6136.03");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2817.61,6026.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3290.50,6070.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4135.30,6165.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5283.28,6179.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=493.23,5893.02");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2359.22,5933.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4891.72,5967.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5390.86,5840.61");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=907.71,5618.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1369.51,5673.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1728.94,5738.75");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2123.12,5620.17");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3566.53,5761.61");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3954.26,5693.82");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4404.12,5770.41");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4694.99,5537.46");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5794.50,5714.02");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=604.74,5286.27");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2641.24,5505.59");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3245.83,5453.40");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5285.84,5469.11");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5779.33,5356.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=134.15,5177.87");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=963.25,5185.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1547.19,5056.46");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2061.77,5227.45");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3198.58,5055.71");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3810.71,5126.91");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4284.82,5222.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5123.74,5143.49");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=586.51,4800.07");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2745.23,4984.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5463.76,4938.48");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=190.87,4544.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1347.98,4620.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2038.47,4616.07");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2423.36,4589.33");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3223.95,4675.59");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3595.10,4596.97");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4395.99,4630.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4794.45,4761.51");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2975.90,4427.26");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3979.09,4401.01");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5077.81,4447.15");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5453.35,4393.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=6.86,4114.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=656.98,4080.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1040.73,4112.07");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3540.45,4134.07");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4538.67,4241.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5006.03,4072.01");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1818.65,3982.99");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2772.45,3871.55");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3245.68,3935.12");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4359.98,3902.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5685.69,4028.61");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5.21,3572.93");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=448.20,3625.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=835.67,3691.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2293.78,3702.35");
    }

}

