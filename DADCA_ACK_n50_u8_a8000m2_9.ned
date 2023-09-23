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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=224.79,7971.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=824.55,7928.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2323.60,7790.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2624.67,7979.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3630.01,7860.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4102.81,7909.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4504.02,7775.29");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5319.76,7882.66");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=428.67,7555.88");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1679.83,7737.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3257.27,7691.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=80.03,7494.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1431.62,7422.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1783.56,7369.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2148.95,7451.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2745.96,7429.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3149.78,7343.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3599.68,7488.28");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4460.56,7413.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4977.48,7342.00");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5351.12,7373.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5709.16,7417.17");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=331.10,7168.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=748.74,7037.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1140.41,7027.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1894.41,7025.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2273.49,7024.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2806.01,7030.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3986.96,7131.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4674.02,7011.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=79.48,6901.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3348.27,6831.20");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5156.74,6939.91");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5686.02,6940.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=711.38,6630.57");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2952.79,6692.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5335.07,6604.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=208.04,6466.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1092.58,6445.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1631.27,6411.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2113.25,6473.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2533.38,6395.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3216.95,6321.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3718.75,6506.11");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4271.59,6344.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4890.70,6359.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5809.04,6391.22");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=429.67,6141.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2047.40,6128.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2706.46,6065.85");
    }

}

