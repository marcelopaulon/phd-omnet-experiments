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

network DADCA_ACK_n50_a8000m2_2
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
        @display("i=misc/sensor;p=2561.94,7780.12");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4071.82,7900.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4594.36,7925.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5258.91,7945.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=69.41,7569.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=478.76,7508.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1609.39,7601.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2206.80,7514.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3104.53,7530.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3616.05,7556.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4383.90,7565.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4783.94,7541.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5475.50,7636.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5908.14,7735.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1070.47,7473.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1379.59,7276.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5112.63,7345.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=134.60,7114.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1034.49,7108.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1980.44,7127.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2345.86,7155.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2859.93,7067.04");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3347.95,7011.52");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3805.47,7079.21");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4420.67,7050.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5987.16,7114.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=660.27,6916.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1623.98,6945.55");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2360.95,6805.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4923.62,6939.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5440.90,6878.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=532.44,6572.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=966.63,6712.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1333.24,6724.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2005.53,6590.72");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3039.51,6661.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3540.67,6606.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4123.06,6588.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4534.25,6713.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5657.92,6534.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1536.40,6320.27");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4517.82,6271.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5026.03,6268.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=285.67,6210.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=670.76,6198.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1129.40,6214.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1990.45,6053.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2586.03,6192.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3066.19,6238.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5863.24,6141.20");
    }

}

