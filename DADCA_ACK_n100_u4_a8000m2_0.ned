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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=4106.07,7779.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4412.50,7960.82");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1085.59,7732.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2099.51,7649.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2449.11,7731.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2939.08,7715.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3423.72,7698.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4820.54,7584.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5787.72,7640.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=898.20,7373.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1819.16,7282.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3063.53,7367.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3444.22,7323.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4118.27,7362.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=45.82,7078.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1415.34,7251.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2519.95,7248.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4752.56,7201.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5385.70,7252.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5895.69,7041.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=548.21,6968.21");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1772.11,6886.26");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2214.54,6989.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3103.99,6837.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3487.81,6967.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3999.60,6916.75");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4635.02,6797.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=287.36,6616.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1152.61,6724.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2534.78,6569.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4171.24,6536.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5277.21,6671.51");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5708.85,6708.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=786.84,6354.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1698.37,6292.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2062.28,6391.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3078.14,6389.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3479.66,6411.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4617.86,6421.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4986.21,6373.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1160.97,6043.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2620.85,6069.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3801.02,6261.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4223.93,6093.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5585.82,6086.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5962.65,6236.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=352.79,5940.66");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=686.97,5775.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1665.57,5917.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2101.48,5869.17");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3061.84,5990.96");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3631.43,5919.24");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4732.56,5998.23");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=405.14,5551.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1296.64,5575.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3293.75,5536.46");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5091.75,5702.70");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5677.45,5678.17");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=846.42,5313.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2611.52,5461.97");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3734.07,5413.21");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4361.15,5474.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4746.11,5318.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=509.88,5166.52");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1605.77,5123.14");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2222.25,5198.11");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2963.46,5262.59");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4037.45,5083.34");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5314.06,5232.58");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5969.05,5183.26");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=291.66,4888.47");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1036.95,4891.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2509.98,4788.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3518.04,4950.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4323.92,4814.17");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5026.91,4850.40");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5621.92,4899.87");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=717.39,4695.54");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1522.25,4717.22");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1874.08,4741.20");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2917.74,4631.20");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3913.06,4736.07");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5353.05,4628.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=78.80,4406.79");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1862.33,4381.96");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2274.27,4412.87");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3903.11,4306.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4487.52,4482.72");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4992.14,4395.33");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5717.16,4524.94");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=883.55,4172.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1537.27,4215.84");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2512.71,4109.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3404.71,4257.19");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4494.13,4114.15");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5400.20,4239.50");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5904.44,4207.06");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=361.38,3992.76");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1958.75,3810.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2932.12,3918.84");
    }

}

