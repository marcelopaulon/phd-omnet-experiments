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

network DADCA_ACK_n50_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=113.74,16.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=28.93,54.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=2.47,112.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=98.44,209.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=164.42,152.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=121.51,306.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=175.13,272.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=203.77,206.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=220.41,317.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=246.92,378.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=299.93,260.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=282.43,197.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=273.47,137.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=274.63,37.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=372.51,29.63");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=320.19,93.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=166.40,80.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=54.67,146.18");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=402.18,154.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=467.26,165.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=542.55,218.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=482.86,262.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=478.57,339.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=422.82,273.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=370.89,315.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=236.32,87.24");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=38.30,270.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=159.30,367.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=466.34,70.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=532.19,94.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=609.35,223.84");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=672.54,276.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=590.06,330.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=418.47,377.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=188.10,7.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=404.15,92.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=15.92,211.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=159.65,450.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=98.63,421.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=37.56,385.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=53.10,480.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=313.05,446.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=257.08,488.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=175.83,525.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=155.32,601.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=72.72,551.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=110.78,643.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1.06,595.87");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=219.68,437.52");
    }

}

