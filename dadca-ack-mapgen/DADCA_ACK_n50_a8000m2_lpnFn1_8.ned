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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor;p=78.58,187.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=138.93,188.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=147.32,276.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=57.67,298.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=11.04,336.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=168.85,86.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=211.30,149.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=229.25,220.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=14.27,124.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=249.77,42.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=339.17,56.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=221.93,336.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=271.63,417.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=308.29,343.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=187.23,410.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=367.97,290.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=450.42,383.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=436.89,446.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=368.72,444.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=558.66,338.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=483.47,291.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=544.86,253.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=347.82,513.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=263.68,520.77");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=452.35,542.89");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=517.87,569.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=511.92,479.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=607.10,474.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=564.46,420.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=644.95,368.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=702.49,349.13");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=618.51,192.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=646.83,257.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=638.94,527.95");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=698.62,499.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=751.72,574.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=716.45,408.95");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=595.19,133.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=580.49,69.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=588.58,8.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=768.53,370.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=792.00,311.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=828.45,452.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=835.44,513.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=894.54,543.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=886.58,604.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=945.86,640.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=956.58,709.16");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=896.62,770.55");
    }

}

