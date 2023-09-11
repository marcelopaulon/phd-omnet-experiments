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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=30.14,155.71");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=87.98,177.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=227.98,166.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=249.56,262.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=176.42,229.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=358.82,243.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=305.17,178.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=69.80,32.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=179.26,3.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=0.36,88.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=46.99,282.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=51.74,350.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=122.54,321.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=6.93,399.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=97.40,437.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=154.99,458.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=442.20,237.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=525.96,185.05");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=605.00,189.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=633.66,105.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=566.42,62.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=392.34,296.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=411.52,174.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=435.80,42.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=498.95,96.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=423.14,104.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=502.34,6.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=340.11,7.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=260.21,10.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=293.01,81.34");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=362.09,129.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=533.85,302.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=511.76,364.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=432.56,397.15");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=461.75,473.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=440.67,558.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=382.33,523.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=332.39,486.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=267.82,530.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=260.84,613.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=501.49,564.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=575.33,581.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=579.70,658.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=618.11,731.16");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=577.47,785.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=668.52,778.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=485.45,789.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=464.00,855.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=492.91,927.62");
    }

}

