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
        @display("i=misc/sensor;p=2379.88,7786.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3033.82,7948.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3492.01,7892.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4391.27,7983.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5360.91,7993.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2039.10,7550.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3239.26,7594.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3821.47,7720.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4838.14,7617.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5212.19,7647.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5658.84,7682.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1550.58,7435.68");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2439.27,7426.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2823.80,7442.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4291.76,7327.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3289.19,7137.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5465.87,7040.90");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5787.78,7239.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1245.95,6820.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1712.45,6981.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2066.33,7003.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2649.55,6979.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3634.44,7005.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4172.93,6772.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4643.99,6936.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5149.16,6814.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=923.84,6549.79");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2650.67,6620.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3113.77,6719.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3795.71,6633.13");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4752.49,6592.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5619.31,6654.18");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=346.38,6331.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1711.42,6416.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2204.98,6429.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2756.07,6285.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5513.96,6303.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5883.01,6313.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2445.91,6081.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3229.48,6204.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3643.95,6264.08");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4238.08,6198.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5063.91,6195.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=164.80,5950.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=757.22,5996.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1287.22,5967.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1957.52,5975.43");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4673.46,5911.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5453.04,5815.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5950.14,5797.54");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=471.58,5727.52");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2902.63,5650.56");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3351.28,5718.24");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4278.47,5691.33");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=52.51,5281.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1387.60,5381.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1770.10,5383.28");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2362.95,5410.80");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4254.04,5323.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5038.95,5331.16");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=603.51,5240.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1038.83,5204.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2678.19,5238.47");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3933.16,5101.54");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4577.68,5111.16");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5593.65,5161.84");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1980.11,5009.83");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2422.36,4954.17");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5147.74,4938.00");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5946.42,4869.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=6.06,4567.33");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=372.79,4638.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=844.16,4725.17");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1240.09,4735.49");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1709.90,4764.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2725.89,4588.57");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3207.03,4750.64");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4165.44,4742.73");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4481.00,4581.52");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5601.08,4630.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=727.80,4312.61");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2362.41,4295.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3634.44,4479.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4770.22,4332.62");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5348.50,4373.76");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=158.93,4251.16");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1294.97,4262.05");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1745.53,4143.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3202.98,4240.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4050.41,4226.64");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5681.52,4152.42");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=8.82,3914.56");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=372.58,3849.79");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2260.52,3803.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2804.32,3906.08");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3353.86,3861.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3775.67,3889.83");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4301.86,3923.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4954.76,3987.08");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1010.20,3790.79");
    }

}

