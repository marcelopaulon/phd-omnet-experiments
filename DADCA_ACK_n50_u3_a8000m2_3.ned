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

network DADCA_ACK_n50_u3_a8000m2_3
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
        @display("i=misc/sensor;p=4012.93,7917.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4414.88,7976.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5091.32,7882.07");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5655.23,7936.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=115.16,7608.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=602.28,7528.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1367.39,7610.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2209.23,7533.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2728.95,7670.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3626.88,7726.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4024.31,7539.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4609.31,7631.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5366.72,7564.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=983.55,7402.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3123.79,7495.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3729.00,7349.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4412.27,7314.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4908.38,7268.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5858.80,7410.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=288.86,7183.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=727.33,7022.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1323.62,7255.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1823.24,7158.18");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2588.86,7227.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5321.79,7217.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5821.01,7048.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1184.61,6910.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1550.26,6793.02");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2186.46,6847.04");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2765.93,6914.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3369.15,7004.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3921.16,6894.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4685.61,6920.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=148.34,6671.83");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=895.32,6659.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1868.29,6579.98");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2573.89,6588.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3017.76,6521.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3479.16,6523.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4144.30,6583.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4565.62,6589.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5029.27,6699.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5614.09,6530.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=352.95,6303.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=703.00,6288.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1240.76,6498.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4860.99,6379.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1491.92,6177.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2134.18,6181.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2877.86,6113.83");
    }

}

