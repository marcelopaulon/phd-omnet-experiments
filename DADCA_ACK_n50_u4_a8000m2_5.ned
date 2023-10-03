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

network DADCA_ACK_n50_u4_a8000m2_5
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
        @display("i=misc/sensor;p=339.28,7798.09");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1537.70,7992.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2096.42,7902.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2644.93,7912.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3044.90,7984.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4890.59,7932.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5990.14,7975.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=998.62,7661.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1647.89,7561.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2615.95,7536.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3157.61,7569.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3795.63,7581.35");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4427.67,7672.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5263.84,7680.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5718.82,7709.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=420.03,7346.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2018.12,7378.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4677.45,7288.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5154.12,7321.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=163.89,7032.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=813.80,7050.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2374.63,7215.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3232.75,7023.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3715.71,7185.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4398.02,7045.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5648.74,7051.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1183.92,6935.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1812.37,6765.18");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3960.00,6857.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4992.01,6925.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=536.58,6694.52");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1443.03,6601.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2313.90,6717.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2868.85,6635.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4707.48,6687.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=229.63,6366.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=761.87,6389.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1753.22,6403.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3415.77,6479.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4208.65,6457.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5337.85,6394.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5706.26,6501.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=30.66,6037.11");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1000.73,6081.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2735.56,6203.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4735.47,6206.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5950.25,6108.26");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=490.03,6016.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1749.67,5925.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2197.07,6007.83");
    }

}

