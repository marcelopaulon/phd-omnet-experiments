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

network DADCA_ACK_n50_u3_a8000m2_7
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
        @display("i=misc/sensor;p=2262.61,7939.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2877.74,7980.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4220.83,7809.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5641.26,7990.73");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=555.27,7623.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1192.03,7718.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1557.11,7581.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1901.98,7678.76");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2562.20,7669.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3361.72,7719.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3736.59,7690.51");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4562.26,7620.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4940.56,7730.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2135.79,7263.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3065.96,7431.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5436.28,7376.52");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=471.71,7194.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1072.79,7155.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1610.00,7160.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2858.16,7096.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3549.52,7141.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4032.20,7153.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4470.52,7136.42");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4825.89,7166.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5282.04,7010.66");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=140.40,6942.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=472.90,6765.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1433.72,6847.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1867.08,6894.00");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2512.59,6835.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5623.74,6873.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=927.71,6623.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2125.42,6547.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3079.60,6692.95");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3596.87,6740.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3926.21,6559.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4541.47,6621.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4914.76,6755.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5993.37,6738.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=355.98,6370.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1586.48,6425.87");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2698.98,6384.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3340.25,6426.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5218.85,6447.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5621.48,6486.66");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1140.33,6144.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2277.41,6182.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3201.24,6040.23");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3753.79,6155.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4142.63,6201.82");
    }

}

