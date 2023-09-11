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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=43.96,134.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=44.90,215.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=111.92,191.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=28.96,305.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=82.42,367.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=14.74,414.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=147.44,352.55");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=217.38,399.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=208.24,340.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=301.18,314.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=288.32,387.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=405.56,307.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=474.04,283.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=466.25,351.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=547.56,407.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=620.75,459.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=535.83,470.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=513.98,552.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=449.08,483.75");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=590.35,364.75");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=607.97,270.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=695.77,285.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=611.20,209.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=689.65,167.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=732.21,216.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=667.58,230.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=539.67,140.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=488.07,85.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=538.06,48.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=416.51,136.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=421.29,205.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=385.95,65.25");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=443.57,44.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=304.72,35.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=335.86,131.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=261.71,164.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=367.64,240.33");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=313.51,210.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=665.17,347.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=681.50,406.12");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=715.47,496.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=678.63,546.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=604.14,609.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=698.60,617.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=596.74,715.18");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=644.80,674.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=569.53,841.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=614.23,777.43");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=601.46,928.60");
    }

}

