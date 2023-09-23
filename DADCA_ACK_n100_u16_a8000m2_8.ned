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

network DADCA_ACK_n100_u16_a8000m2_8
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
        @display("i=misc/sensor;p=1704.92,7995.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2810.70,7891.93");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3828.96,7832.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4639.24,7962.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4966.56,7815.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1020.26,7675.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3402.08,7731.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4374.80,7722.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5605.61,7587.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5934.17,7738.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=993.82,7296.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1463.58,7416.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1994.58,7491.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2806.68,7435.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3177.23,7406.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4119.32,7479.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5241.18,7357.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5918.47,7294.95");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=77.96,7248.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2267.69,7217.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3613.59,7249.29");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4589.09,7092.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5709.44,7012.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=222.51,6896.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=786.34,6846.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1408.42,6886.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1785.84,6879.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2648.67,6876.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3018.42,6886.62");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3774.46,6882.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2247.09,6670.36");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3549.89,6588.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4140.65,6760.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5094.31,6706.01");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5711.23,6605.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=474.90,6456.60");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1134.98,6283.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2433.49,6290.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2773.72,6476.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4353.95,6326.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4858.57,6329.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1698.96,6235.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3137.88,6112.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3515.16,6110.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4013.09,6196.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5359.41,6249.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=723.13,6000.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1058.33,5819.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1529.36,5914.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2177.68,5961.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2647.09,5832.92");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4334.93,5863.93");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5035.86,6002.65");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=221.21,5616.22");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=603.78,5625.14");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1824.21,5711.08");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3263.37,5637.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3715.58,5715.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4819.68,5624.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5271.78,5571.64");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5751.19,5670.58");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=388.05,5300.26");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=937.03,5425.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1468.57,5361.10");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2234.44,5459.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2976.92,5387.56");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=89.94,5099.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1102.10,5091.61");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1698.20,5045.40");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2098.97,5075.84");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2587.06,5063.76");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3688.71,5257.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4109.17,5233.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4786.40,5151.22");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5148.15,5105.81");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5532.52,5099.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5873.25,5198.57");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=805.09,4820.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1328.39,4812.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3155.31,5004.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1770.70,4655.28");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2590.82,4610.00");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2981.43,4539.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3404.42,4644.75");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3774.45,4768.19");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4333.24,4713.11");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4829.75,4611.37");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5894.57,4638.69");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1001.87,4446.07");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3852.50,4376.84");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5373.79,4497.65");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=660.02,4147.73");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1598.06,4134.97");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1970.02,4108.34");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2963.74,4049.34");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3386.27,4248.54");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4192.44,4263.84");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5174.99,4097.31");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5811.85,4067.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1318.74,3910.46");
    }

}

