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

network DADCA_ACK_n50_u2_a8000m2_3
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
        @display("i=misc/sensor;p=470.92,7809.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1483.98,7810.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3081.89,7800.49");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4102.00,7837.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4514.54,7975.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4999.75,7890.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5375.54,7763.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=932.31,7613.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3699.06,7682.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=268.36,7445.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1782.52,7258.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2417.76,7477.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3468.10,7373.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4446.44,7445.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4989.03,7319.73");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=682.29,7062.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1233.34,7137.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2172.57,7200.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2516.60,7014.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2820.00,7223.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3733.45,7069.48");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4190.66,7115.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5835.98,7211.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=302.23,6988.77");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2050.05,6828.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2879.55,6815.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3350.57,6785.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4370.72,6771.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4736.25,6782.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5516.60,6932.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=250.48,6569.96");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=682.03,6673.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1189.85,6721.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1760.84,6596.36");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3893.71,6739.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5218.11,6542.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5724.74,6554.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1455.11,6316.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2561.23,6490.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2948.94,6464.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3498.13,6382.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4343.76,6391.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=369.31,6230.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1002.87,6069.32");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1937.00,6253.47");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4806.53,6161.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5619.60,6066.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=73.79,5992.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=477.84,5809.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1536.79,5895.90");
    }

}

