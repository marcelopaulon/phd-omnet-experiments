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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=68.03,7798.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=628.38,7866.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1013.50,7781.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2478.00,7897.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3612.48,7766.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4527.73,7980.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5254.02,7771.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5812.32,7787.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1407.24,7676.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1909.99,7724.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4745.51,7620.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=451.66,7451.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2245.41,7448.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2839.82,7401.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3379.50,7332.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4454.52,7340.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5155.84,7382.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=25.61,7163.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=480.58,7027.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1133.57,7257.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1635.54,7227.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2182.95,7092.52");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5758.63,7208.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=829.05,6897.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1341.84,6784.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1900.82,6869.70");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2562.77,6773.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3581.39,6828.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4245.01,6878.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4724.97,6927.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1993.52,6524.36");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3199.37,6757.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4706.19,6553.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5321.41,6718.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5686.39,6645.35");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=117.85,6492.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=512.13,6273.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1000.77,6444.48");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2343.42,6369.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2795.21,6269.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3210.40,6316.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3718.33,6424.91");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4392.52,6315.01");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5031.08,6413.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5638.15,6274.80");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1306.65,6129.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1693.19,6148.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4071.30,6102.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4807.78,6051.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=225.71,6004.38");
    }

}

