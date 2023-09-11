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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=71.42,166.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=152.11,159.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=245.70,140.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=228.10,212.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=193.24,77.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=201.44,15.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=260.74,46.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=166.02,225.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=27.85,233.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=11.29,358.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=68.37,332.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=141.18,301.38");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=178.63,349.39");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=249.65,345.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=272.31,401.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=278.04,461.07");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=215.38,463.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=130.26,462.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=150.53,522.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=69.63,564.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=228.03,602.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=296.84,533.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=141.06,24.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2.75,28.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=7.78,104.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=12.45,470.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=208.70,543.34");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=294.84,595.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=242.42,670.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=367.03,565.43");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=90.90,412.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=98.34,225.92");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=317.26,172.48");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=340.97,427.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=358.80,495.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=427.08,489.57");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=308.56,687.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=369.28,693.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=438.06,660.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=484.73,733.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=519.07,651.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=588.24,626.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=636.11,683.33");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=523.28,578.62");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=408.09,392.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=478.58,362.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=545.19,362.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=568.15,422.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=488.79,512.93");
    }

}

