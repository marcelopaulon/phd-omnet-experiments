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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=98.05,175.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=101.85,238.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=214.44,173.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=296.52,179.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=276.14,98.34");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=215.53,27.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=153.64,65.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=293.82,6.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=169.49,131.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=346.82,75.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=379.02,155.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=471.61,137.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=375.34,260.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=411.05,333.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=476.62,324.71");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=282.03,263.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=439.37,56.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=524.64,99.97");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=547.41,225.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=597.75,187.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=456.93,264.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=644.08,94.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=682.98,22.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=608.49,18.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=695.06,150.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=776.54,117.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=751.39,209.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=665.58,252.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=560.07,286.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=642.26,319.20");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=577.08,384.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=497.49,391.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=426.09,404.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=376.58,477.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=282.99,394.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=317.28,462.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=225.30,451.41");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=143.51,375.36");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=153.62,291.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=41.60,273.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=313.18,339.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=329.54,545.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=111.44,456.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=133.33,515.88");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=161.65,578.24");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=91.55,633.19");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=50.52,545.89");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=46.75,451.13");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=70.29,386.85");
    }

}

