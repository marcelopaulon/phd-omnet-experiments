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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=154.33,31.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=28.04,100.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=82.78,184.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=155.02,252.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=176.21,335.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=228.87,306.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=320.56,270.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=88.03,267.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=52.44,354.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=122.07,376.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=55.97,462.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=41.67,522.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=110.51,554.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=159.86,508.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=201.14,577.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=243.53,634.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=75.68,649.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=133.48,624.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=247.07,530.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=329.14,544.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=390.76,594.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=391.88,429.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=474.84,465.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=446.51,554.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=532.64,581.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=536.97,517.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=611.62,579.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=673.50,555.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=634.03,490.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=691.58,421.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=607.54,430.00");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=535.15,395.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=592.13,364.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=581.04,275.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=667.14,307.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=632.46,233.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=671.11,186.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=597.43,136.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=523.54,159.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=451.41,129.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=372.05,139.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=613.19,42.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=705.94,59.55");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=741.37,128.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=797.12,81.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=823.99,162.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=835.82,259.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=765.59,218.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=780.63,290.15");
    }

}

