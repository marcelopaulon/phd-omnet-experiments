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

network DADCA_ACK_n100_a8000m2_0
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
        @display("i=misc/sensor;p=1889.88,7836.50");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2281.75,7754.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2883.54,7960.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3307.02,7914.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3751.12,7979.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4198.46,7775.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=401.50,7708.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1016.30,7701.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2940.03,7555.33");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4808.52,7507.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5489.98,7566.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1526.06,7357.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1923.46,7308.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3585.97,7336.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4051.18,7390.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=437.40,7037.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=986.53,7103.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2583.17,7077.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4647.13,7091.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5151.31,7138.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1566.68,6882.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2034.31,6935.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3259.64,6961.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3588.40,6719.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4029.83,6773.29");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5017.83,6745.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5822.51,6930.31");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=481.48,6526.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1200.47,6489.39");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1742.65,6501.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2153.12,6484.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2788.48,6494.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3221.20,6511.12");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5711.93,6560.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3641.41,6230.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5239.26,6311.02");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5843.02,6209.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=168.97,6167.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=674.55,6044.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1204.23,6007.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1590.00,6132.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2338.33,6102.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2731.19,6048.99");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4196.50,5974.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4683.82,6185.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1636.98,5710.43");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2018.18,5864.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3211.35,5801.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4610.89,5694.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5244.61,5827.62");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5671.02,5687.32");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1081.18,5645.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2809.48,5512.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3799.54,5536.39");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4280.73,5497.35");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=523.40,5376.51");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2277.19,5189.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3450.60,5365.63");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4864.26,5381.63");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5220.61,5162.74");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5938.73,5195.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=153.56,5031.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=927.51,5096.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1756.63,5140.87");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2942.39,5001.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3608.71,4973.57");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4322.26,5064.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4762.10,4966.43");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=592.03,4881.78");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1554.68,4674.70");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3286.00,4778.85");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4149.93,4733.46");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5349.38,4713.70");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=65.12,4470.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2024.85,4634.14");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2357.26,4444.84");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2698.38,4592.03");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3089.58,4413.84");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4354.76,4382.06");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4773.86,4585.68");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5882.83,4633.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=836.09,4337.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1508.10,4237.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3892.13,4334.48");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5480.01,4305.11");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=509.98,3984.48");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=928.38,3915.18");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1490.72,3869.24");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1954.87,3890.69");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2355.03,3955.81");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2943.85,4048.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3311.19,4105.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4174.79,4047.53");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5038.50,3949.93");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5772.71,3994.48");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2826.49,3688.61");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3862.83,3688.11");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4512.83,3700.77");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5446.94,3670.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=416.90,3596.21");
    }

}

