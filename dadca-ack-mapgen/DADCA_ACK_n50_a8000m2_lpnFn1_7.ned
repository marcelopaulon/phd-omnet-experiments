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
        @display("i=misc/sensor,p=149.95,33.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=224.13,129.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=142.83,153.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=25.33,140.57");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=73.50,200.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=137.49,227.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=203.04,265.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=270.71,212.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=373.44,248.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=347.83,167.74");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=380.16,113.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=445.29,80.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=508.76,105.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=473.24,156.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=568.37,169.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=648.52,208.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=650.10,84.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=691.82,40.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=753.78,28.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=843.03,43.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=817.01,115.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=914.59,105.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=741.58,185.71");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=765.44,242.58");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=767.67,319.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=793.70,394.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=751.44,475.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=707.68,432.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=606.80,428.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=565.58,519.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=492.37,522.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=446.11,561.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=377.19,580.51");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=303.79,622.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=248.58,548.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=165.03,548.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=134.76,493.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=13.11,451.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=29.38,535.18");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=309.50,553.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=223.34,699.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=155.19,717.64");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=137.28,635.21");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=242.22,785.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=295.93,863.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=364.30,788.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=430.31,758.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=386.34,690.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=293.85,737.69");
    }

}

