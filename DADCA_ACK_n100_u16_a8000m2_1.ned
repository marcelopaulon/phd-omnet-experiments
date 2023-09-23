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

network DADCA_ACK_n100_u16_a8000m2_1
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
        @display("i=misc/sensor;p=246.67,7762.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=656.68,7999.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1570.11,7939.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3223.95,7908.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1089.14,7619.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2100.36,7520.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3984.71,7687.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4535.74,7688.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4884.38,7604.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5777.61,7697.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=437.27,7387.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1576.64,7319.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2552.47,7383.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3117.39,7394.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3519.92,7406.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4038.27,7309.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4673.29,7288.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5926.71,7263.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1035.97,7121.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3035.48,7033.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5211.46,7178.74");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=352.94,6931.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=723.13,6858.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1429.07,6769.36");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1837.76,6952.25");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2457.05,6966.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3825.12,6953.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4214.25,7006.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5687.74,6859.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2772.19,6669.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3187.38,6714.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3537.52,6552.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4183.37,6537.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4850.48,6707.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=195.29,6320.34");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1477.09,6421.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1831.37,6290.37");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2362.26,6504.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3163.54,6316.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5316.95,6447.77");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5780.58,6456.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=837.59,6178.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1194.50,6054.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2511.96,6139.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3608.95,6168.04");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4706.88,6192.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5193.15,6110.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=9.00,5779.38");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=476.08,5866.25");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1489.14,5838.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1903.92,5803.24");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3078.11,5864.40");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4025.68,5921.96");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5663.75,5914.55");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1032.98,5677.26");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2582.18,5743.28");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4640.29,5571.33");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=394.30,5508.00");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=860.96,5333.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1534.18,5372.98");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2265.64,5503.38");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3421.28,5405.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4281.05,5492.43");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5241.18,5491.24");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5839.87,5440.50");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=122.08,5191.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1255.71,5154.67");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1790.66,5084.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2515.40,5206.17");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2958.01,5234.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3923.60,5249.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4541.93,5212.45");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5069.37,5136.39");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=696.48,4808.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2385.86,4868.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4201.62,4926.67");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4631.99,4870.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5918.12,4941.46");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=282.73,4550.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1273.66,4605.43");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1986.03,4647.70");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3157.12,4578.82");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3518.27,4685.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3905.11,4723.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4847.13,4554.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5324.72,4578.28");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2621.33,4461.94");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3905.69,4339.77");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5617.97,4359.30");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5976.61,4344.20");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=106.94,4074.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=629.12,4040.46");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1329.91,4197.67");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1723.82,4170.94");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2170.92,4067.62");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3386.91,4286.28");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4496.60,4207.41");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4969.94,4126.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=992.25,4038.68");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2720.76,3942.22");
    }

}

