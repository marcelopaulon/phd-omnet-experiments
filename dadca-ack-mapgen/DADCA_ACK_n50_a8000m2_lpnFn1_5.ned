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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=85.52,22.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=10.71,23.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=186.96,29.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=192.25,90.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=254.96,70.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=278.09,141.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=275.90,207.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=340.93,163.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=236.62,329.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=354.95,263.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=402.33,176.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=371.03,106.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=339.88,52.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=292.97,8.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=303.40,344.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=181.70,360.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=108.35,309.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=69.16,378.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=410.82,397.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=291.08,274.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=398.26,311.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=446.55,261.91");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=460.22,500.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=210.35,147.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=437.31,43.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=493.66,100.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=507.83,196.29");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=581.73,212.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=527.98,317.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=562.66,395.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=599.24,91.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=199.23,229.62");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=117.37,170.67");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=48.09,205.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=119.51,235.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=15.45,104.52");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=232.78,437.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=197.93,497.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=278.43,533.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=376.87,534.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=347.38,621.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=303.23,670.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=248.50,695.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=220.86,626.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=430.45,569.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=412.00,457.92");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=127.14,468.85");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=21.71,301.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=17.02,431.60");
    }

}

