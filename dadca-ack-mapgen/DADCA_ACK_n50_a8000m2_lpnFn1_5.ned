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
        @display("i=misc/sensor,p=191.84,115.99");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=179.26,175.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=88.31,193.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=38.64,24.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=126.80,20.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=109.61,268.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=128.57,332.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=90.06,390.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=13.65,352.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=148.71,434.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=250.28,368.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=309.09,438.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=352.65,505.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=408.01,454.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=244.30,433.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=330.77,606.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=260.47,546.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=392.70,585.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=404.32,718.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=337.06,708.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=277.88,668.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=230.59,727.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=179.89,681.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=189.10,595.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=200.27,532.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=115.27,524.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=130.68,611.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=89.69,463.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=214.38,235.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=184.26,299.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=32.94,428.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=266.33,189.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=380.51,349.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=432.16,266.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=483.83,228.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=446.81,175.87");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=544.72,160.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=486.71,120.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=460.56,51.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=553.44,228.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=519.12,283.60");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=606.13,138.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=610.91,41.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=672.51,85.51");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=519.18,18.89");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=383.50,176.84");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=271.44,126.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=275.97,54.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=24.97,147.02");
    }

}

