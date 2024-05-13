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

network DADCA_ACK_n100_u3_a8000m2_4
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
        @display("i=misc/sensor;p=2147.87,7881.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3133.62,7855.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4660.86,7987.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5385.54,7887.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=100.39,7544.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=506.81,7739.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1121.61,7621.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1465.75,7740.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2773.46,7748.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3699.18,7736.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4051.07,7644.92");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4411.84,7692.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5037.39,7719.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5829.57,7737.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=666.75,7367.29");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1722.09,7387.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2082.74,7263.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2476.68,7280.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3253.03,7476.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4498.49,7302.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5023.47,7358.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=0.67,7164.91");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=346.99,7102.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2978.43,7051.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3822.38,7162.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5427.01,7243.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=958.56,6805.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1577.25,7009.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2189.51,6906.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2603.73,6789.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3649.05,6851.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4318.25,6777.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4752.33,6779.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5127.49,6862.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5962.47,6996.49");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1406.17,6549.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2314.66,6548.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5605.89,6593.64");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=35.48,6474.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=454.66,6355.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1850.71,6407.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3076.34,6401.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5967.11,6484.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=867.79,6178.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2745.77,6068.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3862.59,6189.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4585.67,6245.02");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5027.65,6155.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5504.06,6163.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1440.32,5957.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2074.60,5825.80");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3082.95,5842.07");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3443.77,5924.92");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3789.97,5844.70");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1170.60,5551.08");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4461.65,5646.09");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5008.72,5768.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5419.64,5623.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5745.07,5754.70");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1706.70,5457.41");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2668.54,5475.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3028.59,5480.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3708.94,5480.12");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4006.18,5295.20");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5008.30,5342.42");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5993.80,5428.49");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1859.44,5132.22");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2238.64,5134.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2844.94,5150.57");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3365.10,5169.37");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4637.57,5171.23");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1489.31,4956.41");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3907.77,4868.88");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4346.23,4836.51");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5457.09,5002.69");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2437.41,4771.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3016.41,4597.82");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3448.48,4780.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4934.06,4693.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1297.00,4373.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1825.72,4482.02");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4430.28,4371.44");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5482.42,4499.57");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5888.61,4345.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1882.30,4090.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2490.22,4209.31");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3812.58,4205.09");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4878.48,4172.61");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5587.20,4101.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=523.86,3939.52");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2373.11,3809.28");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3220.79,3938.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4083.11,3831.46");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5092.59,3878.05");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5943.51,3824.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1180.31,3786.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1528.38,3663.47");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1985.66,3601.65");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2829.74,3758.68");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3634.88,3735.33");
    }

}

